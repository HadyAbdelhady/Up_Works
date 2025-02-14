import 'package:bookly/core/utils/styles.dart';
import 'package:flutter/material.dart';
import 'best_seller_books_list_view.dart';
import 'custom_app_bar.dart';
import 'featured_books_list_view.dart';

class HomeViewBody extends StatelessWidget {
  const HomeViewBody({super.key});

  @override
  Widget build(BuildContext context) {
    return const CustomScrollView(
      slivers: [
        SliverToBoxAdapter(
          child: SafeArea(
            child: Column(
              crossAxisAlignment: CrossAxisAlignment.start,
              children: [
                CustomAppBar(),
                FeaturedBooksListView(),
                SizedBox(
                  height: 24,
                ),
                Padding(
                  padding: EdgeInsets.symmetric(
                    horizontal: 20,
                    vertical: 8,
                  ),
                  child: Text(
                    'Best Seller',
                    style: Styles.textStyle18,
                  ),
                ),
              ],
            ),
          ),
        ),
        SliverFillRemaining(
          child: BestSellerBooksListView(),
        )
      ],
    );
  }
}
